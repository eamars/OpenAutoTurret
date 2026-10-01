# Independent ADR-002.2 yaw review — results bundle

Read `ADR-002.2-independent-review.md`, then `EVIDENCE.md`.

## Contents

- `audit.py`: read-only, executable audit of all 25 physical journals and the latest fit's support.
- `reproduced/audit_summary.json`: compact executed results.
- `reproduced/audit_runs.csv`: per-run raw decoder, coordinate, current-timing and gyro diagnostics.
- `reproduced/fit_window_inventory.csv`: all 29 fitted segments.
- `reproduced/run_log.txt`: script output from this review.
- `candidate13_velocity.png`: native-gyro illustration of bursts and stalls.
- `candidate13_raw_motion.csv`: native projected gyro at its sample timestamps; encoder/reference interpolated onto those timestamps for illustration only.
- `candidate13_current.csv`: current feedback at encoder feedback times, successful command ZOH, and command ZOH shifted by the original 60.593 ms assumption.

The time axis of the Candidate13 illustration CSVs is relative to `yaw_control_begin`.
Its reference contains an initial hold before the moving command.

## Reproduce the audit

```sh
python -m pip install -r requirements.txt
python audit.py /path/to/ADR-002.2-yaw-physics-math-20261001.zip --out my_audit
```

Python 3.10 or later is required. The script reads the archive without extracting it.
It does not run hardware, load original project code, write to the original archive,
or generate deployment gains. A completed audit process is not a qualification pass.

The current-delay results in `reproduced/` use a common comparison interval for all
lags, starting 150 ms after the first successful TX. They identify only descriptive
telemetry alignment. They do not establish physical Iq/current semantics, precise
actuator delay or torque bandwidth.

The original data-only archive is not duplicated in this bundle. It must be supplied
to rerun the audit. The native optimizer and controller implementation were not supplied,
so this script is not a reproduction of the complete native fitter or controller.

## Qualification

Yaw remains UNQUALIFIED. Candidate14 was NEVER physically run. No pitch commissioning
or formal production validation is established. No replacement model is qualified
by this independent review. All architectural changes are proposals requiring the
stated identification and physical verification workflow.
