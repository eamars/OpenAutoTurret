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

## When a process dies (ADR-008, 2026-11-10)

In normal operation (hardware or `--sim`, no `--frames`) the launcher supervises:

| Process | If it exits |
|---|---|
| **controld** | The stack stops as before (the rest are stopped, controld has parked if it could) and the launcher exits **1**. The boot service then restarts it after 10 s, in **SHUTDOWN** (owner, 2026-11-10). |
| **visiond, webd, calibd, the IMU capture** | Started again by the launcher after 1, 2, 4 … s, capped at 60 s, reset by 10 minutes up; never given up on. controld keeps running and the station **stays homed**. The page's STOP is away for the seconds webd takes to return; `stop` over SSH is not. |

- `launcher.log` (or the user journal under the boot service) and the service's own log say
  `<name> exited (<status>) after N s; restart K in D s`.
- A stop someone asked for (`stop`, a deploy, `systemctl --user stop`, a signal) exits 0 and is not
  restarted.
- A launcher killed outright runs no cleanup. Under the boot service, systemd then sends every child
  TERM (`KillMode=control-group`), so controld parks, and restarts the unit. It does not run ExecStop
  in this case, and under the old `KillMode=mixed` it SIGKILLed controld, which never parked (station,
  2026-11-10). Outside the boot service, `stop` finds the children in `$RUN/children` and stops them,
  controld first.
- The launcher's own lines under the boot service are in the system journal:
  `journalctl _SYSTEMD_USER_UNIT=ota-station.service` (the station keeps no user journal).
- Commissioning, captures and perception-only runs are not supervised: a child that ends ends the
  run, as before.

**Evidence that survives a reboot:** `run/evidence/<utc>-<cause>/` beside `run/state`, on the SD
card. It holds the cause, the exit status, the last MiB of each log that matters, and the journal
tail under the boot service. One bundle is written per controld exit, and per service restart at
most once per 10 minutes per service. The newest 20 are kept, 200 MiB at most.

**Storage budgets in the run dir (a tmpfs, i.e. RAM).** Each log is capped in place: copied to
`<name>.1`, then truncated, every 10 s if needed.

| File | Cap (×2 with `.1`) |
|---|---|
| `controller.log` | 64 MiB |
| `vision.log` | 16 MiB |
| other logs | 8 MiB |
| `resources.ndjson` | 4 MiB |

- `logs-history/` keeps 10 rounds and at most 256 MiB.
- The run dir as a whole has a budget of 768 MiB. Over it, history goes first, then the `.1` parts.
- `OTA_LOG_CAP_BYTES`, `OTA_HISTORY_BUDGET_BYTES` and `OTA_RUN_BUDGET_BYTES` override the budgets
  for a test.

**Resources:** `status` prints `resources.json`, the latest of a once-a-minute record. It holds each
process's RSS, threads, open files, uptime, restarts and last exit, plus the run dir's size and the
free space in RAM and on the SD card. `resources.ndjson` keeps the history. A count of restarts that
keeps rising, or an RSS that keeps climbing, is the early warning that the 2026-11-10 stop lacked.

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
