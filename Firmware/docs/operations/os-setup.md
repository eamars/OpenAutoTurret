# OS setup: real-time motor threads, CPU split, UI last

## What this is for

The one-time OS preparation that lets controld's motor threads run real time. Run it on a fresh
station, after an OS reinstall or SD-card swap, after creating a new operator account, and whenever
controld's log says `SCHED_FIFO ... refused`. It is **not** part of a normal deploy: a deploy
changes the release, this changes the OS once, and the launcher reapplies everything else on each
start.

The reasons (measured contention, owner ruling 2026-10-03) are in
[`../STATION_OPERATIONS.md`](../STATION_OPERATIONS.md), section "Scheduling: what runs real time".

## Where the work happens

On the station, by the **owner**, with sudo, once. Agents operate as `eamars` without sudo
(`AGENTS.md`), so an agent prepares and verifies this step but does not run the `--apply` line; it
asks the owner to. The `--check` line needs no sudo and is the agent's.

Everything after the OS step happens without privileges, every start, from the launcher
(`scripts/run_application.sh`) and from controld itself:

| What | Who applies it | Class |
|---|---|---|
| controld's `rx-can0` (GM6020 feedback; **the yaw servo steps on it**) | controld | SCHED_FIFO 48 |
| `rx-can1` (CyberGear feedback) | controld | SCHED_FIFO 47 |
| `pitch-servo` (1 kHz host position loop) | controld | SCHED_FIFO 46 |
| `yaw-guard`, `cg-watchdog` | controld | SCHED_FIFO 45 |
| `controld` (the main thread: the 200 Hz loop) | controld | SCHED_FIFO 44 |
| `vision-accept`, `vision-rx` (target input) | controld | normal, nice 0 |
| `web-accept`, `web-client`, `imu-observer`, `log-writer` | controld | nice 10 |
| controld as a process | launcher | CPU 3 alone |
| visiond (perception and tracking) | launcher | CPUs 0-2, nice 0 |
| imu-bno085 (observe-only) | launcher | CPUs 0-2, nice 10 |
| webd (the web UI) | launcher | CPUs 0-2, nice 19 |

The kernel's own threaded IRQ handlers for the two SPI CAN controllers run at FIFO 50, **above**
every controld thread: they are what deliver the feedback and carry the commands. The grant is
therefore capped at 49.

## The commands

On the station. The script ships in every release; there is no `run/current` link, so take the
newest release directory (any release from 5676fbb on carries it):

```bash
R=~/workspace/OpenAutoTurret/run/releases/$(ls -t ~/workspace/OpenAutoTurret/run/releases | head -1)
bash "$R/Firmware/tools/station_os_setup.sh" --check
sudo bash "$R/Firmware/tools/station_os_setup.sh" --apply
```

In the examples below, `Firmware/tools/...` means that same path inside the release.

Then **log out and in again** (a new SSH session; limits are read at login), and restart the stack
from that session: `bash Firmware/scripts/run_application.sh stop` then `start`, or a deploy with
`--activate`, which opens its own new session.

Optional, each its own decision:

```bash
sudo bash Firmware/tools/station_os_setup.sh --apply --isolate-cpu3          # then reboot
sudo bash Firmware/tools/station_os_setup.sh --apply --performance-governor   # only on a stiff supply
sudo bash Firmware/tools/station_os_setup.sh --apply --headless               # no desktop
```

- `--isolate-cpu3` adds `isolcpus=3 irqaffinity=0-2` to `/boot/firmware/cmdline.txt` (backed up
  beside it). The kernel then keeps every other task and movable IRQ off controld's CPU.
- `--performance-governor` installs `ota-cpu-governor.service`. **Not before the new PSU**: the 5 V
  rail through the slip ring droops under load already (`throttled=0x50000`), and a fixed top clock
  draws more.
- `--headless` boots to `multi-user.target`. The desktop costs about 200 MB of RAM and no measurable
  CPU, so this is tidiness, not performance.

Launcher overrides, for diagnosis only (not for normal deployment): `OTA_RT=0` keeps every
controld thread SCHED_OTHER; `OTA_CPU_PIN=0` leaves all CPUs unpinned; `OTA_CONTROL_CPUS` and
`OTA_APP_CPUS` move the split.

### Start at boot (one time)

```bash
sudo bash "$R/Firmware/tools/station_os_setup.sh" --apply --autostart    # then reboot
```

This is the only sudo step for boot. It turns on linger for the operator, so their systemd user
manager runs without a login, and gives that manager the same real-time grant as above
(`/etc/systemd/system/user@<uid>.service.d/ota-realtime.conf`).

Everything else is the deploy's job, every time, without sudo:
- `deploy_station.py --activate` points `run/current` at the new release.
- It installs or refreshes the user unit `~/.config/systemd/user/ota-station.service` from
  `scripts/ota-station.service.in`, and enables it.
- Once the manager carries the grant, it starts the stack through the unit.

At boot the unit runs `scripts/station_boot.sh` from `run/current`. That script waits for can0,
can1 and `/dev/hailo0`, then runs the launcher in the foreground in **SHUTDOWN**: up and reachable,
motors off, waiting for HOME. Stopping the unit (or shutting the Pi down) runs the launcher's
controlled stop.

Verify: `systemctl --user status ota-station`, `bash "$R/Firmware/tools/station_os_setup.sh" --check`
(linger, the manager's grant, `run/current`), and after a reboot the HUD reading
`SHUTDOWN · MOTORS OFF · MENU › HOME`.

## What it proves

`--check` reporting the limits file and `pam_limits active` proves a new login will be granted
SCHED_FIFO up to 49 and unlimited mlock. After a restart, the grant is in force when controld's log
has one line per motor thread, `thread rx-can0 (tid N): SCHED_FIFO 48` and so on, plus
`memory locked`. Confirm it from outside:

```bash
ps -T -o tid,cls,rtprio,ni,psr,comm -p "$(pgrep -x controld)"
ps -o pid,ni,psr,args -p "$(pgrep -f perception.visiond)","$(pgrep -f web.webd.app)"
```

Motor threads show `FF` with their priority and `PSR 3`; web/log threads show `NI 10`; webd `NI 19`.
The effect is in the controller log's once-a-second line: `loop: ... worst=` and the new
`step work p50/p99/worst`, and in the count of `supervisor: DERATE reason='control-loop cycle
overrun'` lines per hour.

## What it does not prove

That the control loop now meets its deadline. That is measured, not inferred: compare the overrun
count per hour and `worst=` against the before figures in the runbook section. A grant also does not
make any thread real time by itself: the launcher must pass `OTA_RT=1` (it does by default), and a
thread not in the table above stays SCHED_OTHER by design.

## When it fails

- **`SCHED_FIFO 48 refused (Operation not permitted; RLIMIT_RTPRIO=0)`**: the stack was started
  from a session older than the limits file, or by a process that did not pass through PAM (a
  systemd *user* service ignores limits.d). Start it from a new SSH login.
- **`mlockall refused`**: the same cause for `memlock`. The station still runs; pages can be
  swapped.
- **A controld thread shows `FF` but is not in the table**: it was created by a real-time thread
  before it named its class. controld creates real-time threads with `SCHED_RESET_ON_FORK` and every
  other class drops real time on entry, so this is a defect, not a setting.
- **Pinning missing (`PSR` varies)**: `taskset` absent or fewer than four CPUs; the launcher skips
  pinning rather than fail.
- **The station froze for a second at a time**: a SCHED_FIFO thread that spins is held to 95% of a
  CPU per second by the kernel (`sched_rt_runtime_us=950000`). Leave that cap in place; it is the
  only thing between a looping motor thread and an unresponsive station.
