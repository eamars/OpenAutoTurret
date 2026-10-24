# ADR-005 stage 2: runs from the page — build plan and status

Stage 2 of [ADR-005](ADR-005.md) (D7, D8): VERIFY, RE-RUN and NEW's measurement run on the station
from MENU › CALIBRATION, in a CALIBRATING phase inside controld, orchestrated by `calibd`.
Stage 1 (records, switching, bore trims) is built and verified on the station (2026-10-24, `0309d0c`).

## What the code already gives (read 2026-10-24)

- **The pipeline is station-agnostic.** `commission.py` asks its station for one session at a time,
  `station.run(axis, manifest, label)`, and gets a journal dict back (`journal.yaw` / `journal.pitch`).
  `Station` does it over ssh with commissiond; `SimStation` with `servo-sim`. calibd adds a third,
  `ControldStation`, and the identification, design and scoring code stays as it is.
- **The journal interface is narrow.**
  - Yaw: `header`, `yaw_control_begin`, `yaw_feedback`, `yaw_current_tx`, `servo_cycle`,
    `servo_learned`, `servo_gains`, `footer`, and pitch `register_read` for the idle-axis record.
  - Pitch: `header`, `pitch_trial_window`, `pitch_trial_approach`, `pitch_trial`,
    `pitch_trial_gains`, `footer`, and yaw `can_rx` for the idle-axis record.
- **A session is a manifest.** It holds the servo or loop parameters, a 50 Hz reference table, an
  optional gain schedule and excitation, and its limits. commissiond's two executors
  (`yaw_control_session.cpp`, the trial in `homing_session.cpp`) step the same `axis::Servo` and
  `axis::PositionLoop` that production runs.
- **What production lacks.**
  - Kernel receive timestamps: `SocketCanBus` stamps `rx_ns` in user space; commissiond's
    `TimestampedReceiver` takes the kernel's.
  - A journal writer: commissiond's `Journal` is a single-producer ring with its own disk thread.
  - scipy (`identify.py`, `score.py`). An arm64 wheel exists for the station venv's Python 3.13
    (scipy 1.18.1, checked 2026-10-24); `requirements-station.txt` is installed by every deploy.

## The protocol: page → controld ← calibd

calibd is a second client of controld's web socket (`max_clients` 8) and reads its telemetry.

| Command | From | When | Effect |
|---|---|---|---|
| `verify_calibration` | page | Homed, IMU fresh, no switch | enters CALIBRATING, run kind VERIFY |
| `run_calibration <kind>` | page | the same; not on #1 | RERUN, RERUN+ACCURACY, RERUN+CROSSTALK; NEW comes from `new_calibration_record` |
| `stop_calibration` | page | CALIBRATING | ends the session now, both axes held, run STOPPED, MANUAL |
| `calibration_session <run> <axis> <manifest>` | calibd | CALIBRATING, no session | controld runs one session and writes its journal beside the manifest |
| `calibration_note <run> <text>` | calibd | CALIBRATING | the run view's last line |
| `calibration_finish <run> <verdict> [dir]` | calibd | CALIBRATING, no session | installs the accepted parts from `dir` (if any), HOMEs, resumes the interrupted mode |

- **calibd must answer.** A run that sees no calibd command for 15 s, while no session is active, ends
  NOT RUN ("calibd is not answering"). This is a persistence timeout, not a fault.
- **Telemetry `calibration_run`:** run id, kind, state (`waiting`, `session`, `fitting`, `done`,
  `stopped`, `failed`), the steps, the current session (label, axis, elapsed, expected), the last
  note, and the verdict.
- **STOP MOTION and HOLD during a run are STOP.** STOP is the operator's word, so the run ends in
  MANUAL, as STOP does during a swap hold. A failed run, which no one stopped, returns to the mode it
  interrupted (ADR-005 D7, classification).

## Increments

| # | Scope | Proved by | Status |
|---|---|---|---|
| S2.1 | CALIBRATING, the run lifecycle and gates, `calibration_run` telemetry, the journal writer, the **pitch** session executor (yaw held) | unit tests on the sim backend | built 2026-10-24 |
| S2.2 | the **yaw** session executor (pitch held), kernel RX timestamps on the yaw bus | unit tests; production servo unchanged in the sim | — |
| S2.3 | calibd and `ControldStation`, scipy, the launcher; the page's run view and STOP | Python tests against a fake controld; the page in the browser | — |
| S2.4 | installing accepted parts (`history/`), HOME and resume; NEW's measurement | unit tests | — |
| S2.5 | VERIFY (`check yaw`), with D8's two fixes (session end status; the pitch gate) | Python tests | — |
| S2.6 | the servo-commissioning card rewritten | doc-tree check | — |

**On the station, with the owner's go only** (no real calibration until the payload is mounted and
the owner says so): pitch identify through commissiond and through controld on the same plant,
comparing journals and fits (ADR-005 risk 1); then the first VERIFY; then a RE-RUN.

## As built

- **S2.1 (2026-10-24).**
  - CALIBRATING is an overlay on the homed hold, like the swap hold: the phase stays `hold`, the
    mode is MANUAL, and telemetry says `calibrating`. A new phase would have needed a second copy of
    the hold's stepping.
  - The session executor is `calibration/calibration_session.*`. The backend's pitch servo thread
    steps it under the production guards (drive running, fresh feedback, the end-stop guard and
    the travel governor).
  - commissiond's `Journal` moved to `capture/journal.*`, which both executables link.
  - A session ends Holding. The control loop then re-anchors MANUAL's hold where the axis stands,
    the backend hands the axis back to production, and the writer thread writes the idle axis's
    record and the footer.
  - A supervisor BRAKE, FAULT-STOP or DISABLE stops the session where it is.

