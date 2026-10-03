#!/usr/bin/env bash
# Boot entry of ota-station.service (docs/operations/os-setup.md, "Start at boot").
#
# Owner ruling 2026-10-03: the station is Homed or Shutdown, and a boot is Shutdown -- web, camera
# and controller up, both motors off, nothing moves until the web's HOME (the way a printer's
# firmware comes up). The service is a user unit, which cannot order itself after the system units
# that bring the hardware up (ota-can-links, hailort), so this waits for what they produce, then
# runs the launcher in the foreground. The unit itself never changes; this script ships in every
# release and is found through run/current, so a deploy can change boot behaviour without sudo.
set -euo pipefail
APP="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

can_up() { ip link show "$1" 2>/dev/null | grep -q 'state UP'; }
for _ in $(seq 1 120); do
  if can_up can0 && can_up can1 && [ -e /dev/hailo0 ]; then break; fi
  sleep 1
done
can_up can0 && can_up can1 || echo 'station_boot: CAN links not up after 120 s; starting anyway (controld will say why)' >&2

export OTA_START_STATE="${OTA_START_STATE:-shutdown}"
exec "$APP/scripts/run_application.sh" run
