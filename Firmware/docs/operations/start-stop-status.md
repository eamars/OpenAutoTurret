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
bash Firmware/scripts/run_application.sh start     # detached; also the no-argument default
bash Firmware/scripts/run_application.sh status     # who owns the stack, and from which checkout
bash Firmware/scripts/run_application.sh stop       # controlled park + full cleanup
```

`start` returns after the children are launched, **not** after readiness — a station that is up and a
station that is ready are different claims, and the difference is tens of seconds of homing.

## What it proves

`stop` printing a clean result proves the launcher's stop path ran: motion cancelled, axes brought
down, children reaped. `status` proves which release and run dir own the stack right now.

## What it does not prove

A clean stop is not stop *qualification*: it shows the sequence ran, not that the envelope held
(§19, and the `DEFERRED_TO_ADR002` items). Note also that `stop` prints `STOP FAILED` while still
stopping cleanly — a known defect on the account, not a reason to reach for `kill`.

Manual/Hold is an **operator override**, not a state to normalize: if the operator parked the
station on purpose — for instance because yaw cannot currently be driven stably — a restart that
silently returns it to AUTO_ROAM is a change of intent made by the wrong party.
