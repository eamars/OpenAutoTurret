# tools/tracking: ADR-003 camera tracking, stage programs

Plan, decisions and status: [`docs/ADR-003/IMPLEMENTATION.md`](../../docs/ADR-003/IMPLEMENTATION.md).
The C++ that the station and the simulator share is [`tracking_core/`](../../tracking_core/):
- `estimator.*`: the target estimator;
- `level1.*`: the Level-1 reference generator;
- `tracker.*`: pixel to reference sample;
- `tracking_config.*`: the parameter asset;
- `tracking_sim.*`: the closed-loop simulator, which uses the ADR-002.2 servos and plant models
  from `axis_control_core/`.

| File | Role |
|---|---|
| `tracking.py` | The stage commands. `stage1` runs the 14 scenarios, the checks and the FF/FB comparison, and writes a report to `run/tracking/`. |
| `sim.py` | Runs `tracking-sim`, and reads the station geometry (`calibration/`), the servo assets (`config/servo/`) and the tracking parameters (`config/tracking/`). |
| `scenarios.py` | The 14 ADR-003 scenarios as simulator requests: target truth relative to the start view, camera model and injections. |
| `evaluate.py` | Run metrics against independent truth: framing error at exposure, blur, lag, settle, the error layers (e_goal, e_track, e_servo), and the reference's consistency and bounds. |
| `stage1.py` | One property check per scenario, with bounds from design quantities (H, 4/λ, pixel noise, braking distance). Reported performance. |
| `tests/` | Stage 1 as pytest: every check passes, the target FF removes moving lag, and the simulation is deterministic. |

Run from the workstation (the native build is in `run/adr0022-local/firmware`, the venv is shared
with `servo_commission`):

```bash
wsl.exe -e bash -c "cd /mnt/c/workspace/OpenAutoTurret/Firmware/tools/tracking && ../../../run/servo-commission/venv/bin/python tracking.py stage1"
```

Parameters: [`config/tracking/tracking_prior.json`](../../config/tracking/tracking_prior.json)
holds stage 1's explicit synthetic values, each with its source. Stage 2 writes the measured and
solved asset, `config/tracking/tracking.json`. The photography budget, including the calibrated
yaw servo share, is [`config/tracking/photography_spec.json`](../../config/tracking/photography_spec.json).
