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
- Normal startup is AUTO_ROAM → target tracking → AUTO_ROAM after loss.
  Manual/Hold is an explicit web override; do not persist trial speed/mode
  overrides into normal deployment unintentionally.
- Deploy committed source with `Firmware/tools/deploy_station.py`. It preserves
  the dirty Pi checkout and builds a separate release; `--activate` additionally performs a
  controlled stop/start after verification. **Compilation happens on the deployment host, not on
  the station** — the launcher's own `deploy` action is a different operation (it builds the
  checkout it is standing in), so read
  [the deploy card](Firmware/docs/operations/deploy.md) before deploying from a machine you did not
  build the last release on, and read `Firmware/tools/doc_tree_check.py`'s rules if you add a card.
- Stop through the launcher; do not use broad process kills, bypass homing,
  overwrite retained calibration, or run legacy controller/camera services beside it.
- Python dependencies belong in project-local virtual environments. Never put
  credentials, virtual environments or runtime captures into commits.
- Run the test suite where the hardware is. A workstation container has no
  station and no `sudo`: run `ctest -E "retained_homing"` there for fast
  feedback (that one test writes `/dev/shm`), and let `deploy_station.py`
  run the full suite, hardware included, on the station itself. Do not
  escalate privileges to make the local suite green.
