# Start, stop, status: the one supervised stack

## What this is for

The whole stack — controller, perception, web, IMU capture — is one supervised unit under
`Firmware/scripts/run_application.sh`. Do not start a controller or a camera beside it, and do not
broad-kill: the launcher owns the process group and knows how to park.

## Where the work happens

On the station, as `eamars`, without sudo. The stack lives in the release directory it was started
from; its runtime files are in the run dir (`/tmp/ota-stack-<uid>` by default, override
`OTA_RUN_DIR`), which is where logs, the frame tap, the stream manifest and the IMU trace are.

## The commands

```bash
bash Firmware/scripts/run_application.sh start     # detached, SHUTDOWN; also the no-argument default
bash Firmware/scripts/run_application.sh start --home   # detached, homes at once (OTA_START_STATE=homed)
bash Firmware/scripts/run_application.sh status     # who owns the stack, and from which checkout
bash Firmware/scripts/run_application.sh stop       # controlled park + full cleanup
```

**Two states (owner ruling 2026-10-03): Homed or Shutdown.** A hardware `start` is Shutdown: web,
camera and controller up, both motors off, nothing moves until the web's MENU > HOME (which homes,
then AUTO_ROAM). `--home` starts homed. A boot starts Shutdown through the boot service (see the
[OS setup card](os-setup.md), "Start at boot"), and `deploy_station.py --activate` restores the state
it found (`--start-state homed|shutdown` overrides). `--sim` still homes at start.

`start` returns after the children are launched, **not** after readiness — a station that is up and a
station that is ready are different claims, and the difference is tens of seconds of homing.

## What it proves

`stop` printing a clean result proves the launcher's stop path ran: motion cancelled, axes brought
down, children reaped. `status` proves which release and run dir own the stack right now.

## What it does not prove

A clean stop is not stop *qualification*: it shows the sequence ran, not that the envelope held
(§19, and the `DEFERRED_TO_ADR002` items).

**What a stop does to the motors (owner, 2026-10-03).** A stop of a homed turret, from `stop`, a
deploy or the boot service at power-off, runs the web's SHUTDOWN: yaw to 0, pitch onto its rest stop,
then both motors off. It reports `Stopped: SHUT DOWN (pitch on its rest stop, ...)`, typically after
10-20 s. A turret that cannot park (not homed, faulted) gets the older controlled stop, which releases
pitch where it is. A station already shut down reports `Stopped: already shut down`; that case used
to print `STOP FAILED`, and no longer does.

Manual/Hold is an **operator override**, not a state to normalize: if the operator parked the
station on purpose — for instance because yaw cannot currently be driven stably — a restart that
silently returns it to AUTO_ROAM is a change of intent made by the wrong party.

## Sending a command while it runs

`POST /api/command` takes a body of the shape `{"command": ...}`. A payload shaped some other way is
rejected by the request model with a validation list naming `body.command` -- that is a 422 from the
web layer, not a controller refusal, and the two mean different things to whoever is debugging.

The operator's mode override survives a stack restart only by accident: a homed activation ends in
AUTO_ROAM by design, so if the operator parked the station on purpose, put it back and say so. A
station that was shut down stays shut down across a deploy.
