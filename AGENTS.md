# Station operation

**Entry point: [`Firmware/docs/README.md`](Firmware/docs/README.md)** — the documentation map, with
one card per operation under [`Firmware/docs/operations/`](Firmware/docs/operations/). Read the card
for what you are about to do *before* touching the station; a card's second section always says which
machine the work happens on, which is the fact people skip and then act on an assumption.

[`Firmware/docs/STATION_OPERATIONS.md`](Firmware/docs/STATION_OPERATIONS.md) is the runbook: the
station's current state, safety history, owner rulings and measured limits — the reasons behind the
cards. Dated as-built reports are historical.

- Operate as `eamars@rpi-turret`, without `sudo`, using the existing SSH key.
- Use `Firmware/scripts/run_application.sh` for the entire stack: `deploy`,
  `check`, `start` (also the no-argument default), `status`, `stop`.
- The station is in one of two states, **Homed** or **Shutdown** (owner ruling 2026-10-03).
  - A boot, and a plain launcher `start`, is Shutdown: web, camera and controller are up, both
    motors are off, and nothing moves until the web's MENU > HOME.
  - A deploy returns the station to the state it found.
  - Once homed, normal operation is AUTO_ROAM → target tracking → AUTO_ROAM after loss.
  - SURVEILLANCE (owner ruling 2026-10-05) is the operator's alternative to AUTO_ROAM: face a saved
    watch point, track, return to it after a loss. HOME still ends in AUTO_ROAM. The watch point
    (`run/state/watch_point.json`) is the one operator setting that persists by design.
  - Manual/Hold is an explicit web override. Do not persist trial speed/mode overrides into normal
    deployment unintentionally; the web's speed settings last until a restart by design.
- Deploy committed source with `Firmware/tools/deploy_station.py`. It preserves
  the dirty Pi checkout and builds a separate release; `--activate` additionally performs a
  controlled stop/start after verification. **Compilation happens on the deployment host, not on
  the station** — the launcher's own `deploy` action is a different operation (it builds the
  checkout it is standing in), so read
  [the deploy card](Firmware/docs/operations/deploy.md) before deploying from a machine you did not
  build the last release on, and read `Firmware/tools/doc_tree_check.py`'s rules if you add a card.
- **Fault vs hold (owner ruling 2026-10-02):** before adding or tuning any guard, trip or watchdog,
  read "Fault, hold, degrade" at the top of
  [`Firmware/docs/STATION_OPERATIONS.md`](Firmware/docs/STATION_OPERATIONS.md):
  - FAULT only for safety hazards, and even then hold the axes energised. De-energising an
    unbalanced load is dangerous.
  - Persistent non-hazards HOLD and recover by themselves.
  - Transients only degrade.
  - Every threshold needs margin and a persistence time.
- Stop through the launcher; do not use broad process kills, bypass homing,
  overwrite retained calibration, or run legacy controller/camera services beside it.
- Python dependencies belong in project-local virtual environments. Never put
  credentials, virtual environments or runtime captures into commits.
- Run the test suite where the hardware is. A workstation container has no
  station and no `sudo`: run `ctest -E "retained_homing"` there for fast
  feedback (that one test writes `/dev/shm`), and let `deploy_station.py`
  run the full suite, hardware included, on the station itself. Do not
  escalate privileges to make the local suite green.
