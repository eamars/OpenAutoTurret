# Station operation

Read [Firmware/docs/STATION_OPERATIONS.md](Firmware/docs/STATION_OPERATIONS.md)
before deploying, starting, stopping or diagnosing the physical camera station.
It is the current operating runbook; dated as-built reports are historical.

- Operate as `eamars@rpi-turret`, without `sudo`, using the existing SSH key.
- Use `Firmware/scripts/run_application.sh` for the entire stack: `deploy`,
  `check`, `start` (also the no-argument default), `status`, `stop`.
- Normal startup is AUTO_ROAM → target tracking → AUTO_ROAM after loss.
  Manual/Hold is an explicit web override; do not persist trial speed/mode
  overrides into normal deployment unintentionally.
- Deploy committed source with `Firmware/tools/deploy_station.py`. It preserves
  the dirty Pi checkout and builds a separate release. `--activate` additionally
  performs a controlled stop/start after verification.
- Stop through the launcher; do not use broad process kills, bypass homing,
  overwrite retained calibration, or run legacy controller/camera services beside it.
- Python dependencies belong in project-local virtual environments. Never put
  credentials, virtual environments or runtime captures into commits.
