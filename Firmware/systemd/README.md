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

Preferred: `tools/install_station.py` stages the runtime tree, renders the units
for the real install root / user, and then **checks the four agreements that
fail silently** (vision socket on both sides, web socket on both sides,
`--orientation` vs `OTA_VIDEO_ORIENTATION`, and the fact that the camera device
is exclusive), plus the §52/§33 guarantees (`TimeoutStartSec=infinity`,
`KillSignal=SIGINT`, `Restart=on-failure`). It never calls `systemctl`: starting
`turret-control` homes the turret, and that is a supervised decision.

```sh
cd Firmware
# 1. Stage into the install root and render the units for it (dry run first).
python3 -m tools.install_station stage --stage /opt/open_auto_turret
python3 -m tools.install_station stage --stage /opt/open_auto_turret --apply
# 2. Verify the staged tree BEFORE anything is copied to /etc/systemd/system.
python3 -m tools.install_station check --root /opt/open_auto_turret \
    --units /opt/open_auto_turret/systemd
# 3. Install the rendered units and enable (no --now: see the checklist).
sudo cp /opt/open_auto_turret/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable can0.service turret-control.service \
                      turret-vision.service turret-web.service
# 4. Re-check against what systemd actually holds, and verify python deps.
sudo python3 -m tools.install_station check --root /opt/open_auto_turret \
    --units /etc/systemd/system --verify-python
```

`check` is read-only; run it again after any hand edit, and again whenever
`vision_connected=false` shows up with an otherwise healthy-looking daemon
(checklist #3 is almost always the cause, and the tool says so by name).

Hand-typed equivalent, for the record and for a machine without this checkout:

```sh
# 1. Install the firmware to /opt/open_auto_turret: build/ + config/ + the
#    Python packages. The camera/UI processes run on the SYSTEM python —
#    verified on this station by importing in both interpreters:
#      /usr/bin/python3 : picamera2 OK, uvicorn OK, fastapi/numpy/av/PIL/yaml OK
#      .venv/bin/python : picamera2 FAILS (no libcamera), uvicorn MISSING
#    picamera2 cannot be pip-installed into a venv (it binds the system libcamera
#    stack), so "everything in the venv" is not achievable for those two units.
# 2. Copy the units and reload systemd.
sudo cp /path/to/Firmware/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
# 3. Pre-flight — before anything that can move a motor (see the checklist):
sudo systemd-analyze verify /etc/systemd/system/turret-*.service
systemd-analyze critical-chain turret-control.service
ls -l /opt/open_auto_turret/build/control/controld /opt/open_auto_turret/config/turret.yaml
# 4. Enable the required units (turret-log is optional).
sudo systemctl enable can0.service turret-control.service turret-vision.service turret-web.service
# 5. Start (and watch control return to UNHOMED, then home, then safe hold).
sudo systemctl start can0.service
sudo systemctl start turret-control.service
journalctl -u turret-control.service -f      # wait for "homed + at ready pose"
sudo systemctl start turret-vision.service turret-web.service
```

## Pre-flight checklist (P11) — read before the first `systemctl start`

These are the ways a unit can look healthy and still not run the station:

1. **Homing vs `TimeoutStartSec`.** Homing from cold takes minutes (the
   mechanical zero is volatile; both endpoints are re-derived every boot — ~2.5
   min measured). `turret-control.service` therefore uses
   `TimeoutStartSec=infinity`: with a finite value systemd would SIGTERM the
   daemon mid-home, and because a timeout stop is not a "failure",
   `Restart=on-failure` would leave the station silently unhomed.
2. **Interpreter.** `turret-vision` / `turret-web` run `/usr/bin/python3` (see
   step 1 above). `.venv` remains correct for tests and offline tools.
3. **Vision socket agreement.** controld binds `/tmp/ota_vision.sock`; visiond's
   `--socket` must match (or set `OTA_VISION_SOCKET` on BOTH sides). A mismatch
   is silent: telemetry simply reports `vision_connected=false`, forever, with a
   healthy-looking daemon.
4. **Orientation.** `--orientation rotate_180` on vision (and
   `OTA_VIDEO_ORIENTATION=rotate_180` on web): the IMX500 is mounted
   upside-down. Dropping it does not crash anything — it aims 180° wrong.
5. **Detector.** `--detector none` streams without detections on this platform
   (no RPK/Hailo stack), so tracking stays disarmed by design — that is the safe
   outcome, not a bug. `--detector simple` is the P8 bring-up bridge (§10.1's
   real detector replaces it).
6. **The camera is exclusive.** webd's video and visiond cannot both hold the
   IMX500. Leave webd video off while vision runs.
7. **`turret-can-supervisor` watches `can0`,** i.e. the MCP2515 HAT. While the
   yousee USB-CAN adapter is the primary PHY (`can.backend: yousee` in
   `config/turret.yaml`), that unit is a witness aid only — do not enable it and
   expect it to protect the link the motors actually use.
8. **Logs go to journald** (controld logs to stdout; the units don't redirect),
   so `journalctl -u turret-control -f` is the live view. `turret-log.service` is
   only for a file-based layout (`logs/*.log`), which the current runbook does not
   produce — do not expect it to show anything otherwise.

## Notes

- **can0** is an MCP2515 kernel driver (dtoverlay); `can0.service` waits for the
  interface, sets 1 Mbit/s, and brings it UP. If the driver is missing, control
  fails its CAN-open step and stays UNHOMED (no motion).
- **Vision** runs `--real --orientation rotate_180 --detector none` on the
  target (see the checklist for why each flag is there); dropping `--real` gives
  the synthetic, hardware-free development mode.
- **web** and **control** share `/run/ota/controld-web.sock`; `control` creates
  `/run/ota` in its `ExecStartPre`.
- `PrivateTmp` is left off for control + vision because they share
  `/tmp/ota_vision.sock`. If you move that socket to `/run/ota`, `PrivateTmp`
  could be turned on for vision — but change both sides together (checklist #3).
- The units are **templates**: nothing here is enabled by this repo, and no
  `[SW]` test in the queue depends on them. Enabling + boot-verifying them is
  the live P11 item.
