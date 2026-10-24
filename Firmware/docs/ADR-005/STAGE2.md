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
| `calibration_finish <run> <run dir> <verdict>` | calibd | the run in progress, no session; once | installs the accepted parts from `<run dir>/install/` (if any) through the swap hold, then resumes the interrupted mode |

- **calibd must answer.** A run that sees no calibd command for 15 s, while no session is active and
  no finish is being installed, ends NOT RUN ("calibd is not answering"). This is a persistence
  timeout, not a fault.
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
| S2.2 | the **yaw** session executor (pitch held), kernel RX timestamps on the yaw bus | unit tests; production servo unchanged in the sim | built 2026-10-24 |
| S2.3 | calibd and `ControldStation`, scipy, the launcher; the page's run view and STOP | Python tests against a fake controld; the page in the browser | built 2026-10-24 |
| S2.4 | installing accepted parts (`history/`) and resuming; NEW's measurement | unit tests | built 2026-10-24 |
| S2.5 | VERIFY (`check yaw`), with D8's two fixes (session end status; the pitch gate) | Python tests; the first VERIFY on the station | built 2026-10-24 |
| S2.6 | the servo-commissioning card rewritten | doc-tree check | done 2026-10-24 |

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
- **S2.1, found on the station.** The first VERIFY was refused "HOME first" on a homed station.
  `homed_` is false there by design, because a continuous yaw has no endpoint; `position_ready()`
  is the station's "homed". The simulator never showed it, since its yaw is bounded.
- **S2.2 (2026-10-24).**
  - The yaw executor steps its own `axis::Servo` (the manifest's parameters) on every GM6020 frame,
    under production's guards. Its trips (following error, sensor data, 20 ms encoder speed,
    oscillation, temperature, transmit) end the session.
  - When the session ends it sends zero current until production's servo takes yaw back, which is
    one control cycle at most.
  - The speed check uses the encoder only. commissiond also read the gyro; the gyro still gates a
    run, but it does not reach the backend.
  - Every CAN frame now carries the kernel's receipt time (`SO_TIMESTAMPNS`, mapped to the
    monotonic clock), and production's servos step on it too.
  - A session's journal header holds the manifest re-emitted as JSON, as commissiond's does.
  - `journal.py` reads controld's `idle_axis` row. `tools/servo_commission/tests/test_controld_journal.py`
    parses the journals the C++ executors write.
- **S2.3 (2026-10-24).**
  - `tools/servo_commission/calibd.py` waits for a run in CALIBRATING. It runs the kind's
    pipelines (`check`, `pitch`, `yaw`, `calibrate`) with a `ControldStation`, which gives a
    session to controld and returns its journal parsed.
  - A heartbeat note every 5 s keeps controld's 15 s window open through long fits. Notes read
    `<step> | <line>`, which the run view shows.
  - Runs are kept in `<state>/calibration/runs/`. The pipelines write nothing to `config/`
    (`no_write`).
  - `commission.py` gained an injected station, a log hook, the asset taken from the record, and a
    runs root; it is otherwise unchanged.
  - The launcher starts calibd at nice 10, beside webd, and does not wait on it.
  - The page: VERIFY and RE-RUN (with +ACCURACY and +CROSSTALK) are live and take two presses.
    During a run the drawer is the run view (steps, the session with its time, calibd's last
    line, STOP), the mode block reads CALIBRATING, the dock is dimmed and the D-pad hidden. After
    the run the panel shows LAST RUN.
  - `scipy` is in `requirements-station.txt` and in the test venv.
- **Station, 2026-10-24 18:10, the first VERIFY.** It reached the station from the page and its
  yaw probe session started. Then the station faulted, with both motors off and pitch near level.
  Two bugs, both fixed:
  - The web thread prepared the session (parsed the manifest, allocated and fsynced the 64 MB
    journal ring) under `command_mutex_`, which the 200 Hz loop takes every cycle. The loop waited
    160 ms, and the independent motor watchdog tripped. Since then the web thread's preparation runs
    under its own `web_prep_mutex_`, and `command_mutex_` guards only the gate's state and the
    queue.
  - The run was stepped only inside the homed hold, so in Fault it never ended, and its CALIBRATING
    gate refused the HOME that recovers. Since then it is stepped every cycle in every phase, and a
    fault ends it as "failed".
  - Both have tests that fail on the old code (`CalibrationRun.PreparingABigSession…`,
    `CalibrationRun.AFaultDuringASession…`).
- **S2.4 (2026-10-24).**
  - calibd copies the parts of a run whose every pipeline was accepted into `<run>/install/`.
    Anything less installs nothing: a payload set is one set.
  - controld installs them into the record the run measured, on the web thread. It checks them
    first on a staged copy, keeps the replaced files in `history/<run>/`, marks the parts
    `measured`, and notes `last_run` in record.json.
  - It then puts the record back in force through the swap hold and resumes the interrupted mode.
  - **No HOME after a run** (D7.6 said HOME). The homed frame survives a run, because the encoders
    never lose their count, and the new servos go in force the way a switch puts them. The owner's
    ruling for after a run is "the mode it interrupted".
  - A finished VERIFY, or an installed measurement, clears the VERIFY nudge.
- **NEW's measurement.** `new_calibration_record <from> +MEASURE <name>` makes the copy, switches to
  it, and once the switch has settled starts a MEASURE run, resuming the interrupted mode after it.
  STOP during the switch cancels the measurement. From Shutdown the record is made and put in force,
  and the answer says to HOME and then RE-RUN. The NEW dialog offers MEASURE IT NOW when homed.
- **S2.5, D8's fixes.** The pitch gate (`score.gate_pitch`) passed anything. It now fails a run with
  no scored segments, or with a ramp or the walker above 0.5 deg error RMS: ten times the 10-02
  asset's worst, so it catches a loop that does not follow. `check yaw` counts a probe that did not
  end COMPLETE as a failure.
- **Station, 2026-10-24 18:27, the first VERIFY that ran.**
  - From the page, on #1 AS SHIPPED. The yaw probe session ended COMPLETE, calibd scored it on the
    Pi in about 3 s, and the run took 61 s. AUTO_ROAM resumed.
  - Verdict UPDATE_PARAMETERS. The tracking gates pass (speed ratio 0.965–1.004), but the friction
    is 26–50 % below #1's 10-02 asset. That is the drift record #2 (the 10-20 update, 38–62 % lower)
    was measured for.
  - The probe's journal is 52 MB. A full yaw run is about 1 GB, and journal pruning is still to do.
- **CHECK-PITCH (developer only, 2026-10-24).** It is `run_calibration CHECK-PITCH`, not offered on
  the page. It runs `commission.py check pitch` (the pitch use case with the record's pitch asset)
  in CALIBRATING and installs nothing. It exists so controld's pitch executor can be tried on the
  station before the first real RE-RUN: VERIFY is yaw only (D8).
- **Station, 2026-10-24 18:31, VERIFY on #2 FRICTION 10-20.** COMPLETE in 60 s, verdict
  UPDATE_PARAMETERS. The friction drift is −25, −20, −2, −54, +33 and 0 % across the six speed
  plateaus, against #1's uniform −26 to −50 %. So #2 fits better than #1, but fails the 25 % rule at
  two plateaus. Both records pass the tracking gates. #1 was put back in force afterwards.
- **Station, 2026-10-24 18:41, CHECK-PITCH on #1: controld's pitch executor on the hardware.**
  - The window came from controld's own homing (−0.611 to +0.601 rad).
  - The session ended COMPLETE in 60 s with verdict REUSE_EXACT: ramp RMS 0.018–0.022 deg, walker
    0.022 deg, final step error ≤ 0.003 deg. That is at least as good as the asset's 10-02
    validation with commissiond (0.021–0.045, 0.031 and ≤ 0.007 deg).
  - Held yaw moved 1.5 mrad over the session, under the 0.2 deg decoupling record.
  - **Still to run on the hardware:** the pitch identify sweep and its fit, the yaw survey,
    crosstalk, inertia, ladder and circle sessions, and an install. All are parts of a RE-RUN, which
    waits for the owner's payload and go-ahead.
- **Readiness check before the payload, 2026-10-24 19:00–21:30.**
  - STOP during a VERIFY's session, from the page: both axes held, MANUAL, LAST RUN STOPPED.
  - At 375 px the panel's notes ran off the edge (drawer rows are made for one-word states). They
    wrap now; nothing in the panel is wider than the phone.
  - A run that ends without calibd's finish (STOP, a fault, a silent calibd) is recorded as the
    record's LAST RUN by controld. calibd sends no finish after a STOP.
  - **A finish acted before it was accepted.** The web thread installed parts and wrote
    record.json for any finish, and the control thread refused a finish for a run that had ended
    only afterwards. So a late finish (calibd silent for 15 s during a fit, then done) would have
    put parts in the record that never took force. Only the run in progress, once, touches a
    record now. Once its finish is taken in, the run waits for it: the silence rule does not end it,
    and a STOP ends it in MANUAL after the install. A fault in that window still ends it; the parts
    are then in the record and take force at the next switch, and the refusal says so.
    `CalibrationRun.OnlyTheRunInProgressFinishesIntoARecord` fails on the old code.

